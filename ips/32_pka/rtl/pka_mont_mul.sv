// SPDX-License-Identifier: Apache-2.0
// pka_mont_mul.sv - Word-Serial Pipelined FIOS Montgomery Multiplier & Modular Adder/Subtractor
//
// Implements 32x32->64 FIOS Montgomery modular multiplication (1 limb/cycle inner
// pipeline), constant-time modular addition/subtraction, and hardware 5-step
// Newton-Raphson n0_inv = -M[0]^(-1) mod 2^32 computation.
//
// All execution loops depend solely on num_words_i and alu_op_i (zero secret-
// dependent branches or memory access patterns).

module pka_mont_mul
  import pka_pkg::*;
#(
  parameter int unsigned MAX_WORDS = 128,
  parameter int unsigned ADDR_W    = (MAX_WORDS > 1) ? $clog2(MAX_WORDS) : 1
) (
  input  logic              clk_i,
  input  logic              rst_ni,

  input  logic              start_i,
  input  logic              abort_i,
  input  logic              wipe_i,
  input  logic [ADDR_W-1:0] wipe_addr_i,

  input  pka_alu_op_e       alu_op_i,
  input  logic [ADDR_W:0]   num_words_i,   // 1..MAX_WORDS
  input  logic [31:0]       n0_inv_i,
  output logic [31:0]       n0_inv_o,

  // External Operand Read Ports (1-cycle registered read latency expected)
  output logic [ADDR_W-1:0] a_raddr_o,
  input  logic [31:0]       a_rdata_i,
  output logic              b_req_o,
  output logic [ADDR_W-1:0] b_raddr_o,
  input  logic [31:0]       b_rdata_i,
  output logic [ADDR_W-1:0] m_raddr_o,
  input  logic [31:0]       m_rdata_i,

  // External Result Write Port
  output logic              res_we_o,
  output logic [ADDR_W-1:0] res_waddr_o,
  output logic [31:0]       res_wdata_o,

  output logic              busy_o,
  output logic              done_o
);

  typedef enum logic [4:0] {
    ST_IDLE,
    ST_N0_READ_PRE,
    ST_N0_READ_WAIT,
    ST_N0_STEP1,
    ST_N0_STEP2,
    ST_MM_PREP_B,
    ST_MM_PREP_A,
    ST_MM_J0_P1,
    ST_MM_J0_M,
    ST_MM_J0_P2,
    ST_MM_INNER,
    ST_MM_TOP,
    ST_ADD_P0_PRE,
    ST_ADD_P0,
    ST_SUB_P0_PRE,
    ST_SUB_P0,
    ST_SUB_P1_PRE,
    ST_SUB_P1,
    ST_RED_P1_PRE,
    ST_RED_P1,
    ST_RED_P2_PRE,
    ST_RED_P2,
    ST_DONE
  } state_e;

  state_e state_q;

  logic [ADDR_W:0]   n_q;
  logic [ADDR_W-1:0] n_m1_q;
  logic [ADDR_W-1:0] i_q;
  logic [ADDR_W-1:0] j_q;
  logic [ADDR_W-1:0] j_s1_q;
  logic [ADDR_W-1:0] j_s2_q;
  logic              v_s2_q;
  logic              s1_done_q;

  // FIOS pipeline registers
  logic [31:0] b_i_q;
  logic [31:0] m0_q;
  logic [31:0] mj_q;
  logic [31:0] m_q;
  logic [31:0] s1_q;
  logic [31:0] c1_q;
  logic [31:0] c2_q;
  logic [1:0]  t_hi_q;

  // Add/Sub/Red registers
  logic        carry_q;
  logic        borrow_q;
  logic        ge_m_q;

  // Newton-Raphson n0_inv registers
  logic [2:0]  nr_iter_q;
  logic [31:0] nr_x_q;
  logic [31:0] nr_p_q;

  // Internal 1R1W SRAMs: accumulator (T) and diff/sum buffer (D)
  logic              acc_we;
  logic [ADDR_W-1:0] acc_waddr;
  logic [31:0]       acc_wdata;
  logic [ADDR_W-1:0] acc_raddr;
  logic [31:0]       acc_rdata;

  logic              diff_we;
  logic [ADDR_W-1:0] diff_waddr;
  logic [31:0]       diff_wdata;
  logic [ADDR_W-1:0] diff_raddr;
  logic [31:0]       diff_rdata;

  pka_sram_1r1w #(
    .DEPTH  (MAX_WORDS),
    .DATA_W (32)
  ) u_acc (
    .clk_i   (clk_i),
    .we_i    (acc_we),
    .waddr_i (acc_waddr),
    .wdata_i (acc_wdata),
    .raddr_i (acc_raddr),
    .rdata_o (acc_rdata)
  );

  pka_sram_1r1w #(
    .DEPTH  (MAX_WORDS),
    .DATA_W (32)
  ) u_diff (
    .clk_i   (clk_i),
    .we_i    (diff_we),
    .waddr_i (diff_waddr),
    .wdata_i (diff_wdata),
    .raddr_i (diff_raddr),
    .rdata_o (diff_rdata)
  );

  assign busy_o = (state_q != ST_IDLE);
  assign done_o = (state_q == ST_DONE);

  // Combinational read-address generation, internal SRAM control, and datapath arithmetic
  logic [63:0] p1_comb;
  logic [63:0] p2_comb;
  logic [33:0] top_sum_comb;
  logic [32:0] add_sub_comb;
  logic [31:0] t_cur_comb;

  always_comb begin
    a_raddr_o  = '0;
    b_req_o    = 1'b0;
    b_raddr_o  = '0;
    m_raddr_o  = '0;
    acc_raddr  = '0;
    diff_raddr = '0;

    acc_we     = 1'b0;
    acc_waddr  = '0;
    acc_wdata  = 32'd0;

    diff_we    = 1'b0;
    diff_waddr = '0;
    diff_wdata = 32'd0;

    res_we_o    = 1'b0;
    res_waddr_o = '0;
    res_wdata_o = 32'd0;

    t_cur_comb   = (i_q == '0) ? 32'd0 : acc_rdata;
    p1_comb      = 64'd0;
    p2_comb      = 64'd0;
    top_sum_comb = 34'd0;
    add_sub_comb = 33'd0;

    if (wipe_i) begin
      acc_we     = 1'b1;
      acc_waddr  = wipe_addr_i;
      acc_wdata  = 32'd0;
      diff_we    = 1'b1;
      diff_waddr = wipe_addr_i;
      diff_wdata = 32'd0;
    end else begin
      unique case (state_q)
        ST_N0_READ_PRE: begin
          m_raddr_o = '0;
        end

        ST_MM_PREP_B: begin
          b_req_o   = 1'b1;
          b_raddr_o = i_q;
        end

        ST_MM_PREP_A: begin
          a_raddr_o = '0;
          m_raddr_o = '0;
          acc_raddr = '0;
        end

        ST_MM_J0_P1: begin
          p1_comb = {32'd0, t_cur_comb} + (64'(a_rdata_i) * 64'(b_i_q));
        end

        ST_MM_J0_P2: begin
          p2_comb = {32'd0, s1_q} + (64'(m_q) * 64'(m0_q));
          if (n_q > (ADDR_W+1)'(1)) begin
            a_raddr_o = ADDR_W'(1);
            m_raddr_o = ADDR_W'(1);
            acc_raddr = ADDR_W'(1);
          end
        end

        ST_MM_INNER: begin
          if (!s1_done_q) begin
            p1_comb = {32'd0, t_cur_comb} + {32'd0, c1_q} + (64'(a_rdata_i) * 64'(b_i_q));
            if (j_s1_q < n_m1_q) begin
              a_raddr_o = j_s1_q + ADDR_W'(1);
              m_raddr_o = j_s1_q + ADDR_W'(1);
              acc_raddr = j_s1_q + ADDR_W'(1);
            end
          end
          if (v_s2_q) begin
            p2_comb   = {32'd0, s1_q} + {32'd0, c2_q} + (64'(m_q) * 64'(mj_q));
            acc_we    = 1'b1;
            acc_waddr = j_s2_q - ADDR_W'(1);
            acc_wdata = p2_comb[31:0];
          end
        end

        ST_MM_TOP: begin
          top_sum_comb = 34'(t_hi_q) + 34'(c1_q) + 34'(c2_q);
          acc_we       = 1'b1;
          acc_waddr    = n_m1_q;
          acc_wdata    = top_sum_comb[31:0];
        end

        ST_ADD_P0_PRE: begin
          a_raddr_o = '0;
          b_raddr_o = '0;
        end

        ST_ADD_P0: begin
          add_sub_comb = {1'b0, a_rdata_i} + {1'b0, b_rdata_i} + {32'd0, carry_q};
          acc_we       = 1'b1;
          acc_waddr    = j_q;
          acc_wdata    = add_sub_comb[31:0];
          if (j_q < n_m1_q) begin
            a_raddr_o = j_q + ADDR_W'(1);
            b_raddr_o = j_q + ADDR_W'(1);
          end
        end

        ST_SUB_P0_PRE: begin
          a_raddr_o = '0;
          b_raddr_o = '0;
        end

        ST_SUB_P0: begin
          add_sub_comb = {1'b0, a_rdata_i} - {1'b0, b_rdata_i} - {32'd0, borrow_q};
          acc_we       = 1'b1;
          acc_waddr    = j_q;
          acc_wdata    = add_sub_comb[31:0];
          if (j_q < n_m1_q) begin
            a_raddr_o = j_q + ADDR_W'(1);
            b_raddr_o = j_q + ADDR_W'(1);
          end
        end

        ST_SUB_P1_PRE: begin
          acc_raddr = '0;
          m_raddr_o = '0;
        end

        ST_SUB_P1: begin
          add_sub_comb = {1'b0, acc_rdata} + {1'b0, m_rdata_i} + {32'd0, carry_q};
          diff_we      = 1'b1;
          diff_waddr   = j_q;
          diff_wdata   = add_sub_comb[31:0];
          if (j_q < n_m1_q) begin
            acc_raddr = j_q + ADDR_W'(1);
            m_raddr_o = j_q + ADDR_W'(1);
          end
        end

        ST_RED_P1_PRE: begin
          acc_raddr = '0;
          m_raddr_o = '0;
        end

        ST_RED_P1: begin
          add_sub_comb = {1'b0, acc_rdata} - {1'b0, m_rdata_i} - {32'd0, borrow_q};
          diff_we      = 1'b1;
          diff_waddr   = j_q;
          diff_wdata   = add_sub_comb[31:0];
          if (j_q < n_m1_q) begin
            acc_raddr = j_q + ADDR_W'(1);
            m_raddr_o = j_q + ADDR_W'(1);
          end
        end

        ST_RED_P2_PRE: begin
          acc_raddr  = '0;
          diff_raddr = '0;
        end

        ST_RED_P2: begin
          res_we_o    = 1'b1;
          res_waddr_o = j_q;
          res_wdata_o = ge_m_q ? diff_rdata : acc_rdata;
          if (j_q < n_m1_q) begin
            acc_raddr  = j_q + ADDR_W'(1);
            diff_raddr = j_q + ADDR_W'(1);
          end
        end

        default: ;
      endcase
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q   <= ST_IDLE;
      n_q       <= '0;
      n_m1_q    <= '0;
      i_q       <= '0;
      j_q       <= '0;
      j_s1_q    <= '0;
      j_s2_q    <= '0;
      v_s2_q    <= 1'b0;
      s1_done_q <= 1'b0;
      b_i_q     <= 32'd0;
      m0_q      <= 32'd0;
      mj_q      <= 32'd0;
      m_q       <= 32'd0;
      s1_q      <= 32'd0;
      c1_q      <= 32'd0;
      c2_q      <= 32'd0;
      t_hi_q    <= 2'd0;
      carry_q   <= 1'b0;
      borrow_q  <= 1'b0;
      ge_m_q    <= 1'b0;
      nr_iter_q <= 3'd0;
      nr_x_q    <= 32'd0;
      nr_p_q    <= 32'd0;
      n0_inv_o  <= 32'd0;
    end else if (wipe_i) begin
      state_q   <= ST_IDLE;
      i_q       <= '0;
      j_q       <= '0;
      v_s2_q    <= 1'b0;
      s1_done_q <= 1'b0;
      b_i_q     <= 32'd0;
      m0_q      <= 32'd0;
      mj_q      <= 32'd0;
      m_q       <= 32'd0;
      s1_q      <= 32'd0;
      c1_q      <= 32'd0;
      c2_q      <= 32'd0;
      t_hi_q    <= 2'd0;
      carry_q   <= 1'b0;
      borrow_q  <= 1'b0;
      ge_m_q    <= 1'b0;
      nr_iter_q <= 3'd0;
      nr_x_q    <= 32'd0;
      nr_p_q    <= 32'd0;
      n0_inv_o  <= 32'd0;
    end else if (abort_i) begin
      state_q <= ST_IDLE;
    end else begin
      unique case (state_q)
        ST_IDLE: begin
          if (start_i) begin
            n_q       <= num_words_i;
            n_m1_q    <= ADDR_W'(num_words_i - (ADDR_W+1)'(1));
            i_q       <= '0;
            j_q       <= '0;
            t_hi_q    <= 2'd0;
            carry_q   <= 1'b0;
            borrow_q  <= 1'b0;
            ge_m_q    <= 1'b0;
            v_s2_q    <= 1'b0;
            s1_done_q <= 1'b0;
            unique case (alu_op_i)
              ALU_N0_INV: begin
                nr_iter_q <= 3'd0;
                nr_x_q    <= 32'd1;
                state_q   <= ST_N0_READ_PRE;
              end
              ALU_MONT_MUL: state_q <= ST_MM_PREP_B;
              ALU_MOD_ADD:  state_q <= ST_ADD_P0_PRE;
              ALU_MOD_SUB:  state_q <= ST_SUB_P0_PRE;
            endcase
          end
        end

        // Hardware 5-step Newton-Raphson for n0_inv = -M[0]^(-1) mod 2^32
        ST_N0_READ_PRE:  state_q <= ST_N0_READ_WAIT;
        ST_N0_READ_WAIT: begin
          m0_q    <= m_rdata_i;
          state_q <= ST_N0_STEP1;
        end
        ST_N0_STEP1: begin
          nr_p_q  <= m0_q * nr_x_q;
          state_q <= ST_N0_STEP2;
        end
        ST_N0_STEP2: begin
          automatic logic [31:0] x_next = nr_x_q * (32'd2 - nr_p_q);
          nr_x_q <= x_next;
          if (nr_iter_q == 3'd4) begin
            n0_inv_o <= 32'd0 - x_next;
            state_q  <= ST_DONE;
          end else begin
            nr_iter_q <= nr_iter_q + 3'd1;
            state_q   <= ST_N0_STEP1;
          end
        end

        // FIOS Montgomery Multiplication
        ST_MM_PREP_B: begin
          state_q <= ST_MM_PREP_A;
        end

        ST_MM_PREP_A: begin
          b_i_q   <= b_rdata_i;
          state_q <= ST_MM_J0_P1;
        end

        ST_MM_J0_P1: begin
          m0_q    <= m_rdata_i;
          s1_q    <= p1_comb[31:0];
          c1_q    <= p1_comb[63:32];
          state_q <= ST_MM_J0_M;
        end

        ST_MM_J0_M: begin
          m_q     <= s1_q * n0_inv_i;
          state_q <= ST_MM_J0_P2;
        end

        ST_MM_J0_P2: begin
          c2_q <= p2_comb[63:32];
          if (n_q == (ADDR_W+1)'(1)) begin
            state_q <= ST_MM_TOP;
          end else begin
            j_s1_q    <= ADDR_W'(1);
            v_s2_q    <= 1'b0;
            s1_done_q <= 1'b0;
            state_q   <= ST_MM_INNER;
          end
        end

        ST_MM_INNER: begin
          if (!s1_done_q) begin
            s1_q   <= p1_comb[31:0];
            c1_q   <= p1_comb[63:32];
            mj_q   <= m_rdata_i;
            j_s2_q <= j_s1_q;
            v_s2_q <= 1'b1;
            if (j_s1_q == n_m1_q) begin
              s1_done_q <= 1'b1;
            end else begin
              j_s1_q <= j_s1_q + ADDR_W'(1);
            end
          end else begin
            v_s2_q <= 1'b0;
          end

          if (v_s2_q) begin
            c2_q <= p2_comb[63:32];
          end

          if (s1_done_q) begin
            state_q <= ST_MM_TOP;
          end
        end

        ST_MM_TOP: begin
          t_hi_q <= top_sum_comb[33:32];
          if (i_q == n_m1_q) begin
            state_q <= ST_RED_P1_PRE;
          end else begin
            i_q     <= i_q + ADDR_W'(1);
            state_q <= ST_MM_PREP_B;
          end
        end

        // Modular Addition Pass 0
        ST_ADD_P0_PRE: begin
          j_q     <= '0;
          carry_q <= 1'b0;
          state_q <= ST_ADD_P0;
        end

        ST_ADD_P0: begin
          carry_q <= add_sub_comb[32];
          if (j_q == n_m1_q) begin
            t_hi_q  <= {1'b0, add_sub_comb[32]};
            state_q <= ST_RED_P1_PRE;
          end else begin
            j_q <= j_q + ADDR_W'(1);
          end
        end

        // Modular Subtraction Passes 0 & 1
        ST_SUB_P0_PRE: begin
          j_q      <= '0;
          borrow_q <= 1'b0;
          state_q  <= ST_SUB_P0;
        end

        ST_SUB_P0: begin
          borrow_q <= add_sub_comb[32];
          if (j_q == n_m1_q) begin
            ge_m_q  <= add_sub_comb[32]; // 1 -> negative, add M in P1 and select u_diff
            state_q <= ST_SUB_P1_PRE;
          end else begin
            j_q <= j_q + ADDR_W'(1);
          end
        end

        ST_SUB_P1_PRE: begin
          j_q     <= '0;
          carry_q <= 1'b0;
          state_q <= ST_SUB_P1;
        end

        ST_SUB_P1: begin
          carry_q <= add_sub_comb[32];
          if (j_q == n_m1_q) begin
            state_q <= ST_RED_P2_PRE;
          end else begin
            j_q <= j_q + ADDR_W'(1);
          end
        end

        // Shared Final Reduction Pass 1 (T - M -> diff) & Pass 2 (select -> res)
        ST_RED_P1_PRE: begin
          j_q      <= '0;
          borrow_q <= 1'b0;
          state_q  <= ST_RED_P1;
        end

        ST_RED_P1: begin
          borrow_q <= add_sub_comb[32];
          if (j_q == n_m1_q) begin
            ge_m_q  <= (t_hi_q >= {1'b0, add_sub_comb[32]});
            state_q <= ST_RED_P2_PRE;
          end else begin
            j_q <= j_q + ADDR_W'(1);
          end
        end

        ST_RED_P2_PRE: begin
          j_q     <= '0;
          state_q <= ST_RED_P2;
        end

        ST_RED_P2: begin
          if (j_q == n_m1_q) begin
            state_q <= ST_DONE;
          end else begin
            j_q <= j_q + ADDR_W'(1);
          end
        end

        ST_DONE: begin
          state_q <= ST_IDLE;
        end

        default: state_q <= ST_IDLE;
      endcase
    end
  end

endmodule : pka_mont_mul
