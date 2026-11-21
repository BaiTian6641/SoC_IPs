// SPDX-License-Identifier: Apache-2.0
// pka_core.sv - Protocol-Agnostic Public-Key Cryptographic Accelerator Core
//
// Features:
//   - Parameterizable MAX_BITS (256..4096, default 4096; runtime NUM_WORDS = 1..MAX_WORDS)
//   - OP_MONT_MUL: R = A * B * R^-1 mod M
//   - OP_MOD_EXP:  Constant-time Montgomery powering ladder R = A^E mod M
//   - OP_MOD_ADD / OP_MOD_SUB: Constant-time modular addition & subtraction
//   - OP_X25519_SCALAR: RFC 7748 Curve25519 / X25519 with clamping & Fermat inversion
//   - OP_ECC_P256_PMUL / PADD / PDBL: NIST P-256 constant-time ladder & RCB complete add
//   - Hardware auto-precomputation of n0_inv and R^2 mod M (optional bypass via PRECOMP_EN=0)
//   - Security: Write-only scalar/exponent bank E, synchronous CMD.WIPE zeroization

module pka_core
  import bus_pkg::*;
  import pka_pkg::*;
#(
  parameter int unsigned MAX_BITS  = 4096,
  parameter int unsigned MAX_WORDS = MAX_BITS / 32,
  parameter int unsigned ADDR_W    = (MAX_WORDS > 1) ? $clog2(MAX_WORDS) : 1
) (
  input  logic     clk_i,
  input  logic     rst_ni,

  input  csr_req_t csr_req_i,
  output csr_rsp_t csr_rsp_o,

  output logic     irq_o
);

  // ---------------------------------------------------------------------------
  // CSR Registers & Status
  // ---------------------------------------------------------------------------
  pka_op_e     ctrl_op_q;
  logic        ctrl_precomp_en_q;
  logic        ctrl_ecc_use_g_q;
  logic        ctrl_irq_en_q;
  logic        ctrl_err_irq_en_q;

  logic [ADDR_W:0] num_words_q;
  logic [12:0]     exp_bits_q;
  logic [31:0]     n0_inv_csr_q;
  logic [31:0]     n0_inv_hw_q;
  logic [31:0]     perf_cycles_q;

  logic                  done_st_q;
  logic                  wipe_done_st_q;
  logic [NUM_ERR_BITS-1:0] err_flags_q;

  // ---------------------------------------------------------------------------
  // Top-Level Sequencer States
  // ---------------------------------------------------------------------------
  typedef enum logic [5:0] {
    ST_IDLE,
    ST_WIPE,
    ST_CHK_MOD_PRE,
    ST_CHK_MOD_WAIT,
    ST_SINGLE_N0_WAIT,
    ST_SINGLE_RUN,
    // ModExp Precomputation
    ST_PRE_N0_WAIT,
    ST_PRE_INIT_T0,
    ST_PRE_DBL_R,
    ST_PRE_COPY_T0_T1,
    ST_PRE_DBL_2P32,
    ST_PRE_COPY_T0_T3,
    ST_PRE_EXP_SQ,
    ST_PRE_EXP_MUL,
    ST_PRE_BYPASS_ONE,
    ST_PRE_CONV_A,
    // ModExp Montgomery Ladder
    ST_EXP_RD_E_PRE,
    ST_EXP_RD_E_WAIT,
    ST_EXP_MUL_CROSS,
    ST_EXP_MUL_SQ,
    ST_EXP_SWAP_PRE,
    ST_EXP_SWAP,
    ST_EXP_FINAL,
    // X25519
    ST_X25_INIT,
    ST_X25_RED_U,
    ST_X25_CONV_U,
    ST_X25_COPY_X1,
    ST_X25_RD_E_PRE,
    ST_X25_RD_E_WAIT,
    ST_X25_STEP_UOP,
    // P256
    ST_P256_INIT,
    ST_P256_CONV_P1X,
    ST_P256_CONV_P1Y,
    ST_P256_CONV_P2X,
    ST_P256_CONV_P2Y,
    ST_P256_RD_E_PRE,
    ST_P256_RD_E_WAIT,
    ST_P256_RCB_UOP,
    ST_P256_COPY_OUT,
    ST_P256_COPY_R0_T567,
    // Shared Constant-Time Slot Swap (2 cycles/word)
    ST_CSWAP_RD,
    ST_CSWAP_WR1,
    ST_CSWAP_WR2,
    // Shared Fermat Inversion + Affine Finish
    ST_INV_PREP,
    ST_INV_SQ,
    ST_INV_MUL,
    ST_X25_FIN_MUL,
    ST_X25_FIN_UNMONT,
    ST_P256_FIN_X,
    ST_P256_FIN_Y,
    ST_P256_FIN_UNMONT_X,
    ST_P256_FIN_UNMONT_Y
  } seq_state_e;

  seq_state_e seq_q, cswap_ret_q;

  logic [ADDR_W:0]   n_eff_q;
  logic [ADDR_W-1:0] n_eff_m1_q;
  logic [ADDR_W-1:0] idx_q;
  logic [12:0]       bit_cnt_q;
  logic [12:0]       bit_idx_q;
  logic [3:0]        n_exp_bit_q;
  logic              e_bit_q;
  logic              swap_q;
  logic              cswap_cond_q;
  pka_slot_e         cswap_sa_q, cswap_sb_q;
  logic [2:0]        cswap_cnt_q;
  logic [31:0]       cswap_va_q, cswap_vb_q;

  logic              ecc_is_p256_q;
  logic              p256_dbl_q;
  logic [5:0]        uop_pc_q;
  logic              p1_nz_q, p2_nz_q;
  logic [255:0]      inv_exp_q;

  wire engine_busy = (seq_q != ST_IDLE);

  // ---------------------------------------------------------------------------
  // CSR & SRAM Window Decode
  // ---------------------------------------------------------------------------
  logic [11:0] csr_addr;
  assign csr_addr = csr_req_i.addr[11:0];

  wire csr_wr_en = csr_req_i.valid && csr_req_i.write && (csr_req_i.wstrb != 4'h0);
  wire csr_rd_en = csr_req_i.valid && !csr_req_i.write;

  wire sel_csr    = (csr_addr[11:8] == 4'h0);
  wire sel_a      = (csr_addr[11:9] == 3'b001); // 0x200..0x3FF
  wire sel_b      = (csr_addr[11:9] == 3'b010); // 0x400..0x5FF
  wire sel_m      = (csr_addr[11:9] == 3'b011); // 0x600..0x7FF
  wire sel_e      = (csr_addr[11:9] == 3'b100); // 0x800..0x9FF (WO)
  wire sel_r      = (csr_addr[11:9] == 3'b101); // 0xA00..0xBFF (RO)
  wire sel_r2     = (csr_addr[11:9] == 3'b110); // 0xC00..0xDFF (RW -> T3)
  wire sel_ecc_qx = (csr_addr[11:5] == 7'b111_0000); // 0xE00..0xE1F
  wire sel_ecc_qy = (csr_addr[11:5] == 7'b111_0001); // 0xE20..0xE3F
  wire sel_ecc_ry = (csr_addr[11:5] == 7'b111_0010); // 0xE40..0xE5F (RO)

  wire sel_sram_rd_bank = sel_a || sel_b || sel_m || sel_r || sel_r2 || sel_ecc_qx || sel_ecc_qy || sel_ecc_ry;
  wire [6:0] csr_word_off = csr_addr[8:2];
  wire csr_word_in_range = ({1'b0, csr_word_off} < 8'(MAX_WORDS));
  wire rd_addr_valid     = (sel_ecc_qx || sel_ecc_qy || sel_ecc_ry) ? 1'b1 : csr_word_in_range;

  logic rd_wait_q;

  // ---------------------------------------------------------------------------
  // ALU Instance & Operand Bank Routing
  // ---------------------------------------------------------------------------
  typedef enum logic [3:0] {
    BK_ZERO  = 4'd0,
    BK_ONE   = 4'd1,
    BK_A     = 4'd2,
    BK_B     = 4'd3,
    BK_R     = 4'd4,
    BK_T0    = 4'd5,
    BK_T1    = 4'd6,
    BK_T2    = 4'd7,
    BK_T3    = 4'd8,
    BK_LAD_R = 4'd9,
    BK_ECC   = 4'd10
  } bank_sel_e;

  logic              alu_start, alu_b_req;
  pka_alu_op_e       alu_op;
  logic [ADDR_W-1:0] alu_a_raddr, alu_b_raddr, alu_m_raddr, alu_res_waddr;
  logic [31:0]       alu_a_rdata, alu_b_rdata, alu_m_rdata, alu_res_wdata;
  logic              alu_res_we, alu_busy, alu_done;
  logic [31:0]       alu_n0_inv_out;

  bank_sel_e         src_a_sel, src_b_sel, dst_sel;
  pka_slot_e         ecc_slot_a, ecc_slot_b, ecc_slot_dst;

  wire [31:0] active_n0_inv =
    (seq_q inside {ST_X25_RED_U, ST_X25_CONV_U, ST_X25_STEP_UOP, ST_X25_FIN_MUL, ST_X25_FIN_UNMONT} ||
     (!ecc_is_p256_q && (seq_q inside {ST_INV_SQ, ST_INV_MUL}))) ? P25519_N0INV :
    (ecc_is_p256_q && (seq_q != ST_IDLE)) ? P256_N0INV :
    (ctrl_precomp_en_q ? n0_inv_hw_q : n0_inv_csr_q);

  pka_mont_mul #(
    .MAX_WORDS (MAX_WORDS),
    .ADDR_W    (ADDR_W)
  ) u_alu (
    .clk_i       (clk_i),
    .rst_ni      (rst_ni),
    .start_i     (alu_start),
    .abort_i     (seq_q == ST_IDLE),
    .wipe_i      (seq_q == ST_WIPE),
    .wipe_addr_i (idx_q),
    .alu_op_i    (alu_op),
    .num_words_i (n_eff_q),
    .n0_inv_i    (active_n0_inv),
    .n0_inv_o    (alu_n0_inv_out),
    .a_raddr_o   (alu_a_raddr),
    .a_rdata_i   (alu_a_rdata),
    .b_req_o     (alu_b_req),
    .b_raddr_o   (alu_b_raddr),
    .b_rdata_i   (alu_b_rdata),
    .m_raddr_o   (alu_m_raddr),
    .m_rdata_i   (alu_m_rdata),
    .res_we_o    (alu_res_we),
    .res_waddr_o (alu_res_waddr),
    .res_wdata_o (alu_res_wdata),
    .busy_o      (alu_busy),
    .done_o      (alu_done)
  );

  // ---------------------------------------------------------------------------
  // Inferable Clock-Only 1R1W Operand, Scalar, Result & Scratch SRAM Banks
  // ---------------------------------------------------------------------------
  logic              we_a, we_b, we_m, we_e, we_r, we_t0, we_t1, we_t2, we_t3;
  logic [ADDR_W-1:0] wa_a, wa_b, wa_m, wa_e, wa_r, wa_t0, wa_t1, wa_t2, wa_t3;
  logic [31:0]       wd_a, wd_b, wd_m, wd_e, wd_r, wd_t0, wd_t1, wd_t2, wd_t3;
  logic [ADDR_W-1:0] ra_a, ra_b, ra_m, ra_e, ra_r, ra_t0, ra_t1, ra_t2, ra_t3;
  logic [31:0]       rd_a, rd_b, rd_m, rd_e, rd_r, rd_t0, rd_t1, rd_t2, rd_t3;

  pka_sram_1r1w #(.DEPTH(MAX_WORDS)) u_sram_a  (.clk_i(clk_i), .we_i(we_a),  .waddr_i(wa_a),  .wdata_i(wd_a),  .raddr_i(ra_a),  .rdata_o(rd_a));
  pka_sram_1r1w #(.DEPTH(MAX_WORDS)) u_sram_b  (.clk_i(clk_i), .we_i(we_b),  .waddr_i(wa_b),  .wdata_i(wd_b),  .raddr_i(ra_b),  .rdata_o(rd_b));
  pka_sram_1r1w #(.DEPTH(MAX_WORDS)) u_sram_m  (.clk_i(clk_i), .we_i(we_m),  .waddr_i(wa_m),  .wdata_i(wd_m),  .raddr_i(ra_m),  .rdata_o(rd_m));
  pka_sram_1r1w #(.DEPTH(MAX_WORDS)) u_sram_e  (.clk_i(clk_i), .we_i(we_e),  .waddr_i(wa_e),  .wdata_i(wd_e),  .raddr_i(ra_e),  .rdata_o(rd_e));
  pka_sram_1r1w #(.DEPTH(MAX_WORDS)) u_sram_r  (.clk_i(clk_i), .we_i(we_r),  .waddr_i(wa_r),  .wdata_i(wd_r),  .raddr_i(ra_r),  .rdata_o(rd_r));
  pka_sram_1r1w #(.DEPTH(MAX_WORDS)) u_sram_t0 (.clk_i(clk_i), .we_i(we_t0), .waddr_i(wa_t0), .wdata_i(wd_t0), .raddr_i(ra_t0), .rdata_o(rd_t0));
  pka_sram_1r1w #(.DEPTH(MAX_WORDS)) u_sram_t1 (.clk_i(clk_i), .we_i(we_t1), .waddr_i(wa_t1), .wdata_i(wd_t1), .raddr_i(ra_t1), .rdata_o(rd_t1));
  pka_sram_1r1w #(.DEPTH(MAX_WORDS)) u_sram_t2 (.clk_i(clk_i), .we_i(we_t2), .waddr_i(wa_t2), .wdata_i(wd_t2), .raddr_i(ra_t2), .rdata_o(rd_t2));
  pka_sram_1r1w #(.DEPTH(MAX_WORDS)) u_sram_t3 (.clk_i(clk_i), .we_i(we_t3), .waddr_i(wa_t3), .wdata_i(wd_t3), .raddr_i(ra_t3), .rdata_o(rd_t3));

  // Mirrored ECC 256-bit Slot Bank (16 slots x 8 words = 128 x 32b) for dual-port reads
  logic        we_ecc;
  logic [6:0]  wa_ecc, ra_ecc_a, ra_ecc_b;
  logic [31:0] wd_ecc, rd_ecc_a, rd_ecc_b;

  pka_sram_1r1w #(.DEPTH(128)) u_sram_ecc_a (.clk_i(clk_i), .we_i(we_ecc), .waddr_i(wa_ecc), .wdata_i(wd_ecc), .raddr_i(ra_ecc_a), .rdata_o(rd_ecc_a));
  pka_sram_1r1w #(.DEPTH(128)) u_sram_ecc_b (.clk_i(clk_i), .we_i(we_ecc), .waddr_i(wa_ecc), .wdata_i(wd_ecc), .raddr_i(ra_ecc_b), .rdata_o(rd_ecc_b));

  // Registered 1-cycle constants to match SRAM read timing
  logic [31:0] one_a_rdata_q, one_b_rdata_q, m_const_rdata_q;
  logic        use_ecc_mod;

  assign use_ecc_mod = (seq_q inside {ST_X25_INIT, ST_X25_RED_U, ST_X25_CONV_U, ST_X25_COPY_X1,
                                      ST_X25_RD_E_PRE, ST_X25_RD_E_WAIT, ST_X25_STEP_UOP,
                                      ST_P256_INIT, ST_P256_CONV_P1X, ST_P256_CONV_P1Y,
                                      ST_P256_CONV_P2X, ST_P256_CONV_P2Y, ST_P256_RD_E_PRE,
                                      ST_P256_RD_E_WAIT, ST_P256_RCB_UOP, ST_P256_COPY_OUT,
                                      ST_P256_COPY_R0_T567, ST_CSWAP_RD, ST_CSWAP_WR1, ST_CSWAP_WR2,
                                      ST_INV_PREP, ST_INV_SQ, ST_INV_MUL, ST_X25_FIN_MUL,
                                      ST_X25_FIN_UNMONT, ST_P256_FIN_X, ST_P256_FIN_Y,
                                      ST_P256_FIN_UNMONT_X, ST_P256_FIN_UNMONT_Y});

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      one_a_rdata_q   <= 32'd0;
      one_b_rdata_q   <= 32'd0;
      m_const_rdata_q <= 32'd0;
    end else begin
      one_a_rdata_q   <= (alu_a_raddr == '0) ? 32'd1 : 32'd0;
      one_b_rdata_q   <= (alu_b_raddr == '0) ? 32'd1 : 32'd0;
      m_const_rdata_q <= get_limb256(ecc_is_p256_q ? P256_P : P25519_P, alu_m_raddr[2:0]);
    end
  end

  // Microcode decode for current sequencer state
  pka_uop_t cur_uop;
  always_comb begin
    if (seq_q == ST_X25_STEP_UOP) begin
      cur_uop = get_x25519_uop(uop_pc_q);
    end else begin
      cur_uop = get_p256_add_uop(uop_pc_q, p256_dbl_q);
    end
  end

  // ALU source/destination muxing
  always_comb begin
    alu_start    = 1'b0;
    alu_op       = ALU_MONT_MUL;
    src_a_sel    = BK_ZERO;
    src_b_sel    = BK_ZERO;
    dst_sel      = BK_R;
    ecc_slot_a   = S_X0;
    ecc_slot_b   = S_X0;
    ecc_slot_dst = S_X0;

    unique case (seq_q)
      ST_SINGLE_N0_WAIT: begin
        alu_op = ALU_N0_INV;
      end
      ST_SINGLE_RUN: begin
        unique case (ctrl_op_q)
          OP_MONT_MUL: alu_op = ALU_MONT_MUL;
          OP_MOD_ADD:  alu_op = ALU_MOD_ADD;
          OP_MOD_SUB:  alu_op = ALU_MOD_SUB;
          default:     alu_op = ALU_MONT_MUL;
        endcase
        src_a_sel = BK_A;
        src_b_sel = BK_B;
        dst_sel   = BK_R;
      end

      ST_PRE_N0_WAIT:  alu_op = ALU_N0_INV;
      ST_PRE_DBL_R:    begin alu_op = ALU_MOD_ADD;  src_a_sel = BK_T0;  src_b_sel = BK_T0;  dst_sel = BK_T0; end
      ST_PRE_DBL_2P32: begin alu_op = ALU_MOD_ADD;  src_a_sel = BK_T1;  src_b_sel = BK_T1;  dst_sel = BK_T1; end
      ST_PRE_EXP_SQ:   begin alu_op = ALU_MONT_MUL; src_a_sel = BK_T3;  src_b_sel = BK_T3;  dst_sel = BK_T3; end
      ST_PRE_EXP_MUL:  begin alu_op = ALU_MONT_MUL; src_a_sel = BK_T3;  src_b_sel = BK_T1;  dst_sel = BK_T3; end
      ST_PRE_BYPASS_ONE: begin alu_op = ALU_MONT_MUL; src_a_sel = BK_ONE; src_b_sel = BK_T3; dst_sel = BK_T0; end
      ST_PRE_CONV_A:   begin alu_op = ALU_MONT_MUL; src_a_sel = BK_A;   src_b_sel = BK_T3;  dst_sel = BK_T1; end

      ST_EXP_MUL_CROSS: begin alu_op = ALU_MONT_MUL; src_a_sel = BK_T0;    src_b_sel = BK_T1;    dst_sel = BK_T2; end
      ST_EXP_MUL_SQ:    begin alu_op = ALU_MONT_MUL; src_a_sel = BK_LAD_R; src_b_sel = BK_LAD_R; dst_sel = BK_T3; end
      ST_EXP_FINAL:     begin alu_op = ALU_MONT_MUL; src_a_sel = BK_T0;    src_b_sel = BK_ONE;   dst_sel = BK_R;  end

      ST_X25_RED_U:     begin alu_op = ALU_MOD_ADD;  src_a_sel = BK_ECC; src_b_sel = BK_ZERO; dst_sel = BK_ECC; ecc_slot_a = S_T0; ecc_slot_dst = S_Y0; end
      ST_X25_CONV_U:    begin alu_op = ALU_MONT_MUL; src_a_sel = BK_ECC; src_b_sel = BK_ECC;  dst_sel = BK_ECC; ecc_slot_a = S_Y0; ecc_slot_b = S_C1; ecc_slot_dst = S_Y0; end

      ST_P256_CONV_P1X: begin alu_op = ALU_MONT_MUL; src_a_sel = BK_ECC; src_b_sel = BK_ECC; dst_sel = BK_ECC; ecc_slot_a = S_X0; ecc_slot_b = S_C1; ecc_slot_dst = S_X0; end
      ST_P256_CONV_P1Y: begin alu_op = ALU_MONT_MUL; src_a_sel = BK_ECC; src_b_sel = BK_ECC; dst_sel = BK_ECC; ecc_slot_a = S_Y0; ecc_slot_b = S_C1; ecc_slot_dst = S_Y0; end
      ST_P256_CONV_P2X: begin alu_op = ALU_MONT_MUL; src_a_sel = BK_ECC; src_b_sel = BK_ECC; dst_sel = BK_ECC; ecc_slot_a = S_X1; ecc_slot_b = S_C1; ecc_slot_dst = S_X1; end
      ST_P256_CONV_P2Y: begin alu_op = ALU_MONT_MUL; src_a_sel = BK_ECC; src_b_sel = BK_ECC; dst_sel = BK_ECC; ecc_slot_a = S_Y1; ecc_slot_b = S_C1; ecc_slot_dst = S_Y1; end

      ST_X25_STEP_UOP,
      ST_P256_RCB_UOP: begin
        unique case (cur_uop.op)
          UOP_MUL: alu_op = ALU_MONT_MUL;
          UOP_ADD: alu_op = ALU_MOD_ADD;
          UOP_SUB: alu_op = ALU_MOD_SUB;
          default: alu_op = ALU_MONT_MUL;
        endcase
        src_a_sel    = BK_ECC;
        src_b_sel    = BK_ECC;
        dst_sel      = BK_ECC;
        ecc_slot_a   = cur_uop.src_a;
        ecc_slot_b   = cur_uop.src_b;
        ecc_slot_dst = cur_uop.dst;
      end

      ST_INV_SQ:            begin alu_op = ALU_MONT_MUL; src_a_sel = BK_ECC; src_b_sel = BK_ECC;  dst_sel = BK_ECC; ecc_slot_a = S_T0; ecc_slot_b = S_T0; ecc_slot_dst = S_T0; end
      ST_INV_MUL:           begin alu_op = ALU_MONT_MUL; src_a_sel = BK_ECC; src_b_sel = BK_ECC;  dst_sel = BK_ECC; ecc_slot_a = S_T0; ecc_slot_b = S_C1; ecc_slot_dst = S_T0; end
      ST_X25_FIN_MUL:       begin alu_op = ALU_MONT_MUL; src_a_sel = BK_ECC; src_b_sel = BK_ECC;  dst_sel = BK_ECC; ecc_slot_a = S_X0; ecc_slot_b = S_T0; ecc_slot_dst = S_T1; end
      ST_X25_FIN_UNMONT:    begin alu_op = ALU_MONT_MUL; src_a_sel = BK_ECC; src_b_sel = BK_ONE;  dst_sel = BK_R;   ecc_slot_a = S_T1; end
      ST_P256_FIN_X:        begin alu_op = ALU_MONT_MUL; src_a_sel = BK_ECC; src_b_sel = BK_ECC;  dst_sel = BK_ECC; ecc_slot_a = S_T5; ecc_slot_b = S_T0; ecc_slot_dst = S_T1; end
      ST_P256_FIN_Y:        begin alu_op = ALU_MONT_MUL; src_a_sel = BK_ECC; src_b_sel = BK_ECC;  dst_sel = BK_ECC; ecc_slot_a = S_T6; ecc_slot_b = S_T0; ecc_slot_dst = S_T2; end
      ST_P256_FIN_UNMONT_X: begin alu_op = ALU_MONT_MUL; src_a_sel = BK_ECC; src_b_sel = BK_ONE;  dst_sel = BK_R;   ecc_slot_a = S_T1; end
      ST_P256_FIN_UNMONT_Y: begin alu_op = ALU_MONT_MUL; src_a_sel = BK_ECC; src_b_sel = BK_ONE;  dst_sel = BK_ECC; ecc_slot_a = S_T2; ecc_slot_dst = S_T6; end
      default: ;
    endcase

    // Trigger alu_start on the first cycle in an ALU-wait state when ALU is idle
    if (!alu_busy && !alu_done) begin
      if (seq_q inside {ST_SINGLE_N0_WAIT, ST_SINGLE_RUN, ST_PRE_N0_WAIT,
                        ST_PRE_DBL_R, ST_PRE_DBL_2P32, ST_PRE_EXP_SQ, ST_PRE_EXP_MUL,
                        ST_PRE_BYPASS_ONE, ST_PRE_CONV_A,
                        ST_EXP_MUL_CROSS, ST_EXP_MUL_SQ, ST_EXP_FINAL,
                        ST_X25_RED_U, ST_X25_CONV_U,
                        ST_P256_CONV_P1X, ST_P256_CONV_P1Y, ST_P256_CONV_P2X, ST_P256_CONV_P2Y,
                        ST_INV_SQ, ST_INV_MUL, ST_X25_FIN_MUL, ST_X25_FIN_UNMONT,
                        ST_P256_FIN_X, ST_P256_FIN_Y, ST_P256_FIN_UNMONT_X, ST_P256_FIN_UNMONT_Y}) begin
        alu_start = 1'b1;
      end else if ((seq_q == ST_X25_STEP_UOP || seq_q == ST_P256_RCB_UOP) && (cur_uop.op != UOP_DONE)) begin
        alu_start = 1'b1;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // SRAM Read/Write Port Arbitration (Idle CSR vs Active Engine)
  // ---------------------------------------------------------------------------
  always_comb begin
    we_a = 1'b0; wa_a = '0; wd_a = 32'd0; ra_a = '0;
    we_b = 1'b0; wa_b = '0; wd_b = 32'd0; ra_b = '0;
    we_m = 1'b0; wa_m = '0; wd_m = 32'd0; ra_m = '0;
    we_e = 1'b0; wa_e = '0; wd_e = 32'd0; ra_e = '0;
    we_r = 1'b0; wa_r = '0; wd_r = 32'd0; ra_r = '0;
    we_t0 = 1'b0; wa_t0 = '0; wd_t0 = 32'd0; ra_t0 = '0;
    we_t1 = 1'b0; wa_t1 = '0; wd_t1 = 32'd0; ra_t1 = '0;
    we_t2 = 1'b0; wa_t2 = '0; wd_t2 = 32'd0; ra_t2 = '0;
    we_t3 = 1'b0; wa_t3 = '0; wd_t3 = 32'd0; ra_t3 = '0;
    we_ecc = 1'b0; wa_ecc = 7'd0; wd_ecc = 32'd0; ra_ecc_a = 7'd0; ra_ecc_b = 7'd0;

    if (!engine_busy) begin
      // CSR accesses when idle
      if (csr_wr_en) begin
        if (csr_word_in_range) begin
          if (sel_a) begin
            we_a = 1'b1; wa_a = ADDR_W'(csr_word_off); wd_a = csr_req_i.wdata;
          end else if (sel_b) begin
            we_b = 1'b1; wa_b = ADDR_W'(csr_word_off); wd_b = csr_req_i.wdata;
          end else if (sel_m) begin
            we_m = 1'b1; wa_m = ADDR_W'(csr_word_off); wd_m = csr_req_i.wdata;
          end else if (sel_e) begin
            we_e = 1'b1; wa_e = ADDR_W'(csr_word_off); wd_e = csr_req_i.wdata;
          end else if (sel_r2) begin
            we_t3 = 1'b1; wa_t3 = ADDR_W'(csr_word_off); wd_t3 = csr_req_i.wdata;
          end
        end
        if (sel_ecc_qx) begin
          we_ecc = 1'b1; wa_ecc = {S_X1, csr_addr[4:2]}; wd_ecc = csr_req_i.wdata;
        end else if (sel_ecc_qy) begin
          we_ecc = 1'b1; wa_ecc = {S_Y1, csr_addr[4:2]}; wd_ecc = csr_req_i.wdata;
        end
      end
      if (csr_rd_en) begin
        if (csr_word_in_range) begin
          ra_a  = ADDR_W'(csr_word_off);
          ra_b  = ADDR_W'(csr_word_off);
          ra_m  = ADDR_W'(csr_word_off);
          ra_r  = ADDR_W'(csr_word_off);
          ra_t3 = ADDR_W'(csr_word_off);
        end
        if (sel_ecc_qx)      ra_ecc_a = {S_X1, csr_addr[4:2]};
        else if (sel_ecc_qy) ra_ecc_a = {S_Y1, csr_addr[4:2]};
        else if (sel_ecc_ry) ra_ecc_a = {S_T6, csr_addr[4:2]};
      end
    end else if (seq_q == ST_WIPE) begin
      // Synchronous zeroization of all banks
      we_a  = 1'b1; wa_a  = idx_q; wd_a  = 32'd0;
      we_b  = 1'b1; wa_b  = idx_q; wd_b  = 32'd0;
      we_m  = 1'b1; wa_m  = idx_q; wd_m  = 32'd0;
      we_e  = 1'b1; wa_e  = idx_q; wd_e  = 32'd0;
      we_r  = 1'b1; wa_r  = idx_q; wd_r  = 32'd0;
      we_t0 = 1'b1; wa_t0 = idx_q; wd_t0 = 32'd0;
      we_t1 = 1'b1; wa_t1 = idx_q; wd_t1 = 32'd0;
      we_t2 = 1'b1; wa_t2 = idx_q; wd_t2 = 32'd0;
      we_t3 = 1'b1; wa_t3 = idx_q; wd_t3 = 32'd0;
      we_ecc = 1'b1; wa_ecc = bit_cnt_q[6:0]; wd_ecc = 32'd0;
    end else begin
      // Engine Read Routing (when alu_b_req=1, B read takes priority so src_a==src_b squaring works!)
      ra_m = alu_m_raddr;

      if (alu_b_req && (src_b_sel == BK_A))        ra_a  = alu_b_raddr;
      else if (src_a_sel == BK_A)                  ra_a  = alu_a_raddr;
      else if (src_b_sel == BK_A)                  ra_a  = alu_b_raddr;

      if (alu_b_req && (src_b_sel == BK_B))        ra_b  = alu_b_raddr;
      else if (src_a_sel == BK_B)                  ra_b  = alu_a_raddr;
      else if (src_b_sel == BK_B)                  ra_b  = alu_b_raddr;

      if (alu_b_req && (src_b_sel == BK_R))        ra_r  = alu_b_raddr;
      else if (src_a_sel == BK_R)                  ra_r  = alu_a_raddr;
      else if (src_b_sel == BK_R)                  ra_r  = alu_b_raddr;

      if (alu_b_req && (src_b_sel == BK_T0 || src_b_sel == BK_LAD_R))      ra_t0 = alu_b_raddr;
      else if (src_a_sel == BK_T0 || src_a_sel == BK_LAD_R)                ra_t0 = alu_a_raddr;
      else if (src_b_sel == BK_T0 || src_b_sel == BK_LAD_R)                ra_t0 = alu_b_raddr;

      if (alu_b_req && (src_b_sel == BK_T1 || src_b_sel == BK_LAD_R))      ra_t1 = alu_b_raddr;
      else if (src_a_sel == BK_T1 || src_a_sel == BK_LAD_R)                ra_t1 = alu_a_raddr;
      else if (src_b_sel == BK_T1 || src_b_sel == BK_LAD_R)                ra_t1 = alu_b_raddr;

      if (alu_b_req && (src_b_sel == BK_T2))       ra_t2 = alu_b_raddr;
      else if (src_a_sel == BK_T2)                 ra_t2 = alu_a_raddr;
      else if (src_b_sel == BK_T2)                 ra_t2 = alu_b_raddr;

      if (alu_b_req && (src_b_sel == BK_T3))       ra_t3 = alu_b_raddr;
      else if (src_a_sel == BK_T3)                 ra_t3 = alu_a_raddr;
      else if (src_b_sel == BK_T3)                 ra_t3 = alu_b_raddr;

      ra_ecc_a = {ecc_slot_a, alu_a_raddr[2:0]};
      ra_ecc_b = {ecc_slot_b, alu_b_raddr[2:0]};

      // Sequencer overrides for copies / E-reads / cswap
      unique case (seq_q)
        ST_CHK_MOD_PRE: begin
          ra_m = '0;
        end
        ST_PRE_INIT_T0: begin
          we_t0 = 1'b1; wa_t0 = idx_q; wd_t0 = (idx_q == '0) ? 32'd1 : 32'd0;
        end
        ST_PRE_COPY_T0_T1: begin
          if (bit_cnt_q[0] == 1'b0) begin
            ra_t0 = idx_q;
          end else begin
            we_t1 = 1'b1; wa_t1 = idx_q; wd_t1 = rd_t0;
          end
        end
        ST_PRE_COPY_T0_T3: begin
          if (bit_cnt_q[0] == 1'b0) begin
            ra_t0 = idx_q;
          end else begin
            we_t3 = 1'b1; wa_t3 = idx_q; wd_t3 = rd_t0;
          end
        end
        ST_EXP_RD_E_PRE:  ra_e = ADDR_W'(bit_idx_q >> 5);
        ST_EXP_SWAP_PRE:  begin ra_t2 = '0; ra_t3 = '0; end
        ST_EXP_SWAP: begin
          if (idx_q < n_eff_m1_q) begin
            ra_t2 = idx_q + ADDR_W'(1);
            ra_t3 = idx_q + ADDR_W'(1);
          end
          we_t0 = 1'b1; wa_t0 = idx_q; wd_t0 = e_bit_q ? rd_t2 : rd_t3;
          we_t1 = 1'b1; wa_t1 = idx_q; wd_t1 = e_bit_q ? rd_t3 : rd_t2;
        end
        default: ;
      endcase

      // X25519 / P256 / CSWAP / Inverter / ALU write overrides
      if (seq_q == ST_X25_INIT) begin
        we_ecc = 1'b0;
        ra_a   = ADDR_W'(idx_q[2:0]);
        unique case (bit_cnt_q[3:0])
          4'd0: begin we_ecc = 1'b1; wa_ecc = {S_C0, idx_q[2:0]}; wd_ecc = get_limb256(P25519_A24_M, idx_q[2:0]); end
          4'd1: begin we_ecc = 1'b1; wa_ecc = {S_C1, idx_q[2:0]}; wd_ecc = get_limb256(P25519_R2_M, idx_q[2:0]);  end
          4'd2: begin we_ecc = 1'b1; wa_ecc = {S_X0, idx_q[2:0]}; wd_ecc = get_limb256(P25519_ONE_M, idx_q[2:0]); end
          4'd3: begin we_ecc = 1'b1; wa_ecc = {S_Z0, idx_q[2:0]}; wd_ecc = 32'd0;                                 end
          4'd4: begin we_ecc = 1'b1; wa_ecc = {S_Z1, idx_q[2:0]}; wd_ecc = get_limb256(P25519_ONE_M, idx_q[2:0]); end
          4'd5: begin ra_a = ADDR_W'(idx_q[2:0]); end
          4'd6: begin
            we_ecc = 1'b1; wa_ecc = {S_T0, idx_q[2:0]};
            wd_ecc = (idx_q[2:0] == 3'd7) ? (rd_a & 32'h7FFF_FFFF) : rd_a;
          end
          default: ;
        endcase
      end else if (seq_q == ST_X25_COPY_X1) begin
        if (bit_cnt_q[0] == 1'b0) begin
          ra_ecc_a = {S_Y0, idx_q[2:0]};
        end else begin
          we_ecc = 1'b1; wa_ecc = {S_X1, idx_q[2:0]}; wd_ecc = rd_ecc_a;
        end
      end else if (seq_q == ST_X25_RD_E_PRE || seq_q == ST_P256_RD_E_PRE) begin
        ra_e = ADDR_W'(bit_idx_q >> 5);
      end else if (seq_q == ST_P256_INIT) begin
        ra_a = ADDR_W'(idx_q[2:0]);
        ra_b = ADDR_W'(idx_q[2:0]);
        ra_ecc_a = {S_X1, idx_q[2:0]};
        ra_ecc_b = {S_Y1, idx_q[2:0]};
        unique case (bit_cnt_q[3:0])
          4'd0: begin we_ecc = 1'b1; wa_ecc = {S_C0, idx_q[2:0]}; wd_ecc = get_limb256(P256_B_M, idx_q[2:0]);   end
          4'd1: begin we_ecc = 1'b1; wa_ecc = {S_C1, idx_q[2:0]}; wd_ecc = get_limb256(P256_R2_M, idx_q[2:0]);  end
          4'd2: begin we_ecc = 1'b1; wa_ecc = {S_X0, idx_q[2:0]}; wd_ecc = 32'd0;                               end
          4'd3: begin we_ecc = 1'b1; wa_ecc = {S_Y0, idx_q[2:0]}; wd_ecc = get_limb256(P256_ONE_M, idx_q[2:0]); end
          4'd4: begin we_ecc = 1'b1; wa_ecc = {S_Z0, idx_q[2:0]}; wd_ecc = 32'd0;                               end
          4'd5: begin
            if (ctrl_op_q == OP_ECC_P256_PMUL && ctrl_ecc_use_g_q) begin
              we_ecc = 1'b1; wa_ecc = {S_X1, idx_q[2:0]}; wd_ecc = get_limb256(P256_GX_M, idx_q[2:0]);
            end
          end
          4'd6: begin
            if (ctrl_op_q == OP_ECC_P256_PMUL && ctrl_ecc_use_g_q) begin
              we_ecc = 1'b1; wa_ecc = {S_Y1, idx_q[2:0]}; wd_ecc = get_limb256(P256_GY_M, idx_q[2:0]);
            end else if (ctrl_op_q == OP_ECC_P256_PMUL) begin
              we_ecc = 1'b1; wa_ecc = {S_X1, idx_q[2:0]}; wd_ecc = rd_a;
            end else begin
              we_ecc = 1'b1; wa_ecc = {S_X0, idx_q[2:0]}; wd_ecc = rd_a;
            end
          end
          4'd7: begin
            if (ctrl_op_q == OP_ECC_P256_PMUL && ctrl_ecc_use_g_q) begin
              we_ecc = 1'b1; wa_ecc = {S_Z1, idx_q[2:0]}; wd_ecc = get_limb256(P256_ONE_M, idx_q[2:0]);
            end else if (ctrl_op_q == OP_ECC_P256_PMUL) begin
              we_ecc = 1'b1; wa_ecc = {S_Y1, idx_q[2:0]}; wd_ecc = rd_b;
            end else begin
              we_ecc = 1'b1; wa_ecc = {S_Y0, idx_q[2:0]}; wd_ecc = rd_b;
            end
          end
          4'd8: begin
            if (ctrl_op_q == OP_ECC_P256_PMUL) begin
              we_ecc = 1'b1; wa_ecc = {S_Z1, idx_q[2:0]}; wd_ecc = p1_nz_q ? get_limb256(P256_ONE_M, idx_q[2:0]) : 32'd0;
            end else begin
              we_ecc = 1'b1; wa_ecc = {S_Z0, idx_q[2:0]}; wd_ecc = p1_nz_q ? get_limb256(P256_ONE_M, idx_q[2:0]) : 32'd0;
            end
          end
          4'd9: begin
            // Write normal-domain 1 so MontMul(1, R2) yields ONE_M for infinity Y
            if (ctrl_op_q == OP_ECC_P256_PMUL && !p1_nz_q) begin
              we_ecc = 1'b1; wa_ecc = {S_Y1, idx_q[2:0]}; wd_ecc = (idx_q[2:0] == 3'd0) ? 32'd1 : 32'd0;
            end else if (ctrl_op_q != OP_ECC_P256_PMUL && !p1_nz_q) begin
              we_ecc = 1'b1; wa_ecc = {S_Y0, idx_q[2:0]}; wd_ecc = (idx_q[2:0] == 3'd0) ? 32'd1 : 32'd0;
            end
          end
          4'd10: begin
            if (ctrl_op_q == OP_ECC_P256_PADD) begin
              we_ecc = 1'b1; wa_ecc = {S_Z1, idx_q[2:0]}; wd_ecc = p2_nz_q ? get_limb256(P256_ONE_M, idx_q[2:0]) : 32'd0;
            end
          end
          4'd11: begin
            if (ctrl_op_q == OP_ECC_P256_PADD && !p2_nz_q) begin
              we_ecc = 1'b1; wa_ecc = {S_Y1, idx_q[2:0]}; wd_ecc = (idx_q[2:0] == 3'd0) ? 32'd1 : 32'd0;
            end
          end
          default: ;
        endcase
      end else if (seq_q == ST_P256_COPY_OUT) begin
        automatic pka_slot_e s_src = (bit_cnt_q[2:1] == 2'd0) ? S_T5 :
                                     (bit_cnt_q[2:1] == 2'd1) ? S_T6 : S_T7;
        automatic pka_slot_e s_dst = p256_dbl_q ?
                                     ((bit_cnt_q[2:1] == 2'd0) ? S_X0 :
                                      (bit_cnt_q[2:1] == 2'd1) ? S_Y0 : S_Z0) :
                                     ((bit_cnt_q[2:1] == 2'd0) ? S_X1 :
                                      (bit_cnt_q[2:1] == 2'd1) ? S_Y1 : S_Z1);
        if (bit_cnt_q[0] == 1'b0) begin
          ra_ecc_a = {s_src, idx_q[2:0]};
        end else begin
          we_ecc = 1'b1; wa_ecc = {s_dst, idx_q[2:0]}; wd_ecc = rd_ecc_a;
        end
      end else if (seq_q == ST_P256_COPY_R0_T567) begin
        automatic pka_slot_e s_src = (bit_cnt_q[2:1] == 2'd0) ? S_X0 :
                                     (bit_cnt_q[2:1] == 2'd1) ? S_Y0 : S_Z0;
        automatic pka_slot_e s_dst = (bit_cnt_q[2:1] == 2'd0) ? S_T5 :
                                     (bit_cnt_q[2:1] == 2'd1) ? S_T6 : S_T7;
        if (bit_cnt_q[0] == 1'b0) begin
          ra_ecc_a = {s_src, idx_q[2:0]};
        end else begin
          we_ecc = 1'b1; wa_ecc = {s_dst, idx_q[2:0]}; wd_ecc = rd_ecc_a;
        end
      end else if (seq_q == ST_CSWAP_RD) begin
        ra_ecc_a = {cswap_sa_q, idx_q[2:0]};
        ra_ecc_b = {cswap_sb_q, idx_q[2:0]};
      end else if (seq_q == ST_CSWAP_WR1) begin
        we_ecc = 1'b1; wa_ecc = {cswap_sa_q, idx_q[2:0]};
        wd_ecc = cswap_cond_q ? rd_ecc_b : rd_ecc_a;
      end else if (seq_q == ST_CSWAP_WR2) begin
        we_ecc = 1'b1; wa_ecc = {cswap_sb_q, idx_q[2:0]};
        wd_ecc = cswap_cond_q ? cswap_va_q : cswap_vb_q;
      end else if (seq_q == ST_INV_PREP) begin
        // bit_cnt_q[1:0]: 0=read Z (Z0 or T7), 1=write C1, 2=write T0=ONE_M
        if (bit_cnt_q[1:0] == 2'd0) begin
          ra_ecc_a = {ecc_is_p256_q ? S_T7 : S_Z0, idx_q[2:0]};
        end else if (bit_cnt_q[1:0] == 2'd1) begin
          we_ecc = 1'b1; wa_ecc = {S_C1, idx_q[2:0]}; wd_ecc = rd_ecc_a;
        end else if (bit_cnt_q[1:0] == 2'd2) begin
          we_ecc = 1'b1; wa_ecc = {S_T0, idx_q[2:0]};
          wd_ecc = get_limb256(ecc_is_p256_q ? P256_ONE_M : P25519_ONE_M, idx_q[2:0]);
        end
      end else if (alu_res_we) begin
        unique case (dst_sel)
          BK_R:   begin we_r   = 1'b1; wa_r   = alu_res_waddr; wd_r   = alu_res_wdata; end
          BK_T0:  begin we_t0  = 1'b1; wa_t0  = alu_res_waddr; wd_t0  = alu_res_wdata; end
          BK_T1:  begin we_t1  = 1'b1; wa_t1  = alu_res_waddr; wd_t1  = alu_res_wdata; end
          BK_T2:  begin we_t2  = 1'b1; wa_t2  = alu_res_waddr; wd_t2  = alu_res_wdata; end
          BK_T3:  begin we_t3  = 1'b1; wa_t3  = alu_res_waddr; wd_t3  = alu_res_wdata; end
          BK_ECC: begin we_ecc = 1'b1; wa_ecc = {ecc_slot_dst, alu_res_waddr[2:0]}; wd_ecc = alu_res_wdata; end
          default: ;
        endcase
      end
    end
  end

  // ALU Operand Read Data Muxes
  always_comb begin
    unique case (src_a_sel)
      BK_ZERO:  alu_a_rdata = 32'd0;
      BK_ONE:   alu_a_rdata = one_a_rdata_q;
      BK_A:     alu_a_rdata = rd_a;
      BK_B:     alu_a_rdata = rd_b;
      BK_R:     alu_a_rdata = rd_r;
      BK_T0:    alu_a_rdata = rd_t0;
      BK_T1:    alu_a_rdata = rd_t1;
      BK_T2:    alu_a_rdata = rd_t2;
      BK_T3:    alu_a_rdata = rd_t3;
      BK_LAD_R: alu_a_rdata = e_bit_q ? rd_t1 : rd_t0;
      BK_ECC:   alu_a_rdata = rd_ecc_a;
      default:  alu_a_rdata = 32'd0;
    endcase

    unique case (src_b_sel)
      BK_ZERO:  alu_b_rdata = 32'd0;
      BK_ONE:   alu_b_rdata = one_b_rdata_q;
      BK_A:     alu_b_rdata = rd_a;
      BK_B:     alu_b_rdata = rd_b;
      BK_R:     alu_b_rdata = rd_r;
      BK_T0:    alu_b_rdata = rd_t0;
      BK_T1:    alu_b_rdata = rd_t1;
      BK_T2:    alu_b_rdata = rd_t2;
      BK_T3:    alu_b_rdata = rd_t3;
      BK_LAD_R: alu_b_rdata = e_bit_q ? rd_t1 : rd_t0;
      BK_ECC:   alu_b_rdata = rd_ecc_b;
      default:  alu_b_rdata = 32'd0;
    endcase

    alu_m_rdata = use_ecc_mod ? m_const_rdata_q : rd_m;
  end

  // ---------------------------------------------------------------------------
  // CSR Read Path (0-wait for CSRs, 1-wait for SRAM banks)
  // ---------------------------------------------------------------------------
  logic [31:0] caps_val;
  assign caps_val = {4'd0, 1'b1, 1'b1, 1'b1, 1'b1, 8'(MAX_WORDS), 16'(MAX_BITS)};

  always_comb begin
    csr_rsp_o       = '0;
    csr_rsp_o.ready = 1'b1;

    if (csr_rd_en) begin
      if (sel_csr) begin
        unique case (csr_addr)
          A_ID:          csr_rsp_o.rdata = PKA_ID_VALUE;
          A_CAPS:        csr_rsp_o.rdata = caps_val;
          A_CTRL:        csr_rsp_o.rdata = {24'd0, ctrl_err_irq_en_q, ctrl_irq_en_q, ctrl_ecc_use_g_q, ctrl_precomp_en_q, ctrl_op_q};
          A_CMD:         csr_rsp_o.rdata = 32'd0;
          A_STATUS:      csr_rsp_o.rdata = {19'd0, err_flags_q, 4'd0, wipe_done_st_q, (|err_flags_q), done_st_q, engine_busy};
          A_ERR_STAT:    csr_rsp_o.rdata = {27'd0, err_flags_q};
          A_NUM_WORDS:   csr_rsp_o.rdata = 32'(num_words_q);
          A_EXP_BITS:    csr_rsp_o.rdata = {19'd0, exp_bits_q};
          A_N0_INV:      csr_rsp_o.rdata = ctrl_precomp_en_q ? n0_inv_hw_q : n0_inv_csr_q;
          A_PERF_CYCLES: csr_rsp_o.rdata = perf_cycles_q;
          A_IRQ_STAT:    csr_rsp_o.rdata = {29'd0, wipe_done_st_q & ctrl_irq_en_q, (|err_flags_q) & ctrl_err_irq_en_q, done_st_q & ctrl_irq_en_q};
          default:       csr_rsp_o.rdata = 32'd0;
        endcase
      end else if (sel_e) begin
        // Write-only scalar/private exponent bank always reads 0
        csr_rsp_o.rdata = 32'd0;
      end else if (sel_sram_rd_bank) begin
        if (engine_busy || !rd_addr_valid) begin
          csr_rsp_o.rdata = 32'd0;
        end else if (!rd_wait_q) begin
          csr_rsp_o.ready = 1'b0;
        end else begin
          if (sel_a)           csr_rsp_o.rdata = rd_a;
          else if (sel_b)      csr_rsp_o.rdata = rd_b;
          else if (sel_m)      csr_rsp_o.rdata = rd_m;
          else if (sel_r)      csr_rsp_o.rdata = rd_r;
          else if (sel_r2)     csr_rsp_o.rdata = rd_t3;
          else                 csr_rsp_o.rdata = rd_ecc_a;
        end
      end
    end
  end

  assign irq_o = ((done_st_q || wipe_done_st_q) && ctrl_irq_en_q) || ((|err_flags_q) && ctrl_err_irq_en_q);

  // ---------------------------------------------------------------------------
  // Helper: Find highest set bit of num_words (1..MAX_WORDS) for binary exp
  // ---------------------------------------------------------------------------
  function automatic logic [3:0] msb_idx(input logic [ADDR_W:0] val);
    automatic logic [3:0] res = 4'd0;
    for (int unsigned k = 0; k <= ADDR_W; k++) begin
      if (val[k]) res = 4'(k);
    end
    return res;
  endfunction

  // ---------------------------------------------------------------------------
  // Main Control & Sequencer FSM
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_op_q         <= OP_MONT_MUL;
      ctrl_precomp_en_q <= 1'b1;
      ctrl_ecc_use_g_q  <= 1'b1;
      ctrl_irq_en_q     <= 1'b0;
      ctrl_err_irq_en_q <= 1'b0;
      num_words_q       <= (ADDR_W+1)'(8);
      exp_bits_q        <= 13'd0;
      n0_inv_csr_q      <= 32'd0;
      n0_inv_hw_q       <= 32'd0;
      perf_cycles_q     <= 32'd0;
      done_st_q         <= 1'b0;
      wipe_done_st_q    <= 1'b0;
      err_flags_q       <= '0;
      rd_wait_q         <= 1'b0;

      seq_q             <= ST_IDLE;
      cswap_ret_q       <= ST_IDLE;
      n_eff_q           <= (ADDR_W+1)'(8);
      n_eff_m1_q        <= ADDR_W'(7);
      idx_q             <= '0;
      bit_cnt_q         <= 13'd0;
      bit_idx_q         <= 13'd0;
      n_exp_bit_q       <= 4'd0;
      e_bit_q           <= 1'b0;
      swap_q            <= 1'b0;
      cswap_cond_q      <= 1'b0;
      cswap_sa_q        <= S_X0;
      cswap_sb_q        <= S_X1;
      cswap_cnt_q       <= 3'd0;
      cswap_va_q        <= 32'd0;
      cswap_vb_q        <= 32'd0;
      ecc_is_p256_q     <= 1'b0;
      p256_dbl_q        <= 1'b0;
      uop_pc_q          <= 6'd0;
      p1_nz_q           <= 1'b0;
      p2_nz_q           <= 1'b0;
      inv_exp_q         <= 256'd0;
    end else begin
      // 1-cycle SRAM read wait state tracker
      if (csr_rd_en && sel_sram_rd_bank && !engine_busy && rd_addr_valid && !rd_wait_q) begin
        rd_wait_q <= 1'b1;
      end else begin
        rd_wait_q <= 1'b0;
      end

      // Performance cycle counter runs whenever engine is executing an arithmetic op
      if (engine_busy && (seq_q != ST_WIPE)) begin
        perf_cycles_q <= perf_cycles_q + 32'd1;
      end

      // CSR Writes
      if (csr_wr_en) begin
        if (sel_csr) begin
          unique case (csr_addr)
            A_CTRL: begin
              if (csr_req_i.wstrb[0]) begin
                if (engine_busy) begin
                  err_flags_q[E_BUSY_WR] <= 1'b1;
                end else begin
                  ctrl_op_q         <= pka_op_e'(csr_req_i.wdata[3:0]);
                  ctrl_precomp_en_q <= csr_req_i.wdata[4];
                  ctrl_ecc_use_g_q  <= csr_req_i.wdata[5];
                end
                ctrl_irq_en_q     <= csr_req_i.wdata[6];
                ctrl_err_irq_en_q <= csr_req_i.wdata[7];
              end
            end

            A_STATUS: begin
              if (csr_req_i.wstrb[0]) begin
                if (csr_req_i.wdata[1]) done_st_q      <= 1'b0;
                if (csr_req_i.wdata[2]) err_flags_q    <= '0;
                if (csr_req_i.wdata[3]) wipe_done_st_q <= 1'b0;
              end
              if (csr_req_i.wstrb[1]) begin
                err_flags_q <= err_flags_q & ~csr_req_i.wdata[8 +: NUM_ERR_BITS];
              end
            end

            A_ERR_STAT: begin
              if (csr_req_i.wstrb[0]) begin
                err_flags_q <= err_flags_q & ~csr_req_i.wdata[NUM_ERR_BITS-1:0];
              end
            end

            A_NUM_WORDS: begin
              if (engine_busy) err_flags_q[E_BUSY_WR] <= 1'b1;
              else             num_words_q <= csr_req_i.wdata[ADDR_W:0];
            end

            A_EXP_BITS: begin
              if (engine_busy) err_flags_q[E_BUSY_WR] <= 1'b1;
              else             exp_bits_q <= csr_req_i.wdata[12:0];
            end

            A_N0_INV: begin
              if (engine_busy) err_flags_q[E_BUSY_WR] <= 1'b1;
              else             n0_inv_csr_q <= csr_req_i.wdata;
            end

            A_CMD: begin
              if (csr_req_i.wdata[1]) begin
                // CMD.WIPE: highest priority synchronous zeroization
                n0_inv_csr_q   <= 32'd0;
                n0_inv_hw_q    <= 32'd0;
                perf_cycles_q  <= 32'd0;
                done_st_q      <= 1'b0;
                wipe_done_st_q <= 1'b0;
                idx_q          <= '0;
                bit_cnt_q      <= 13'd0;
                seq_q          <= ST_WIPE;
              end else if (csr_req_i.wdata[2]) begin
                // CMD.ABORT
                if (engine_busy) begin
                  seq_q                    <= ST_IDLE;
                  err_flags_q[E_ABORTED]   <= 1'b1;
                end
              end else if (csr_req_i.wdata[0]) begin
                // CMD.START
                if (engine_busy) begin
                  err_flags_q[E_BUSY_WR] <= 1'b1;
                end else begin
                  automatic logic is_ecc = (ctrl_op_q == OP_X25519_SCALAR) ||
                                           (ctrl_op_q == OP_ECC_P256_PMUL) ||
                                           (ctrl_op_q == OP_ECC_P256_PADD) ||
                                           (ctrl_op_q == OP_ECC_P256_PDBL);
                  automatic logic bad_op = (ctrl_op_q > OP_ECC_P256_PDBL);
                  automatic logic bad_w  = !is_ecc && ((num_words_q == '0) || (num_words_q > (ADDR_W+1)'(MAX_WORDS)));

                  if (bad_op) begin
                    err_flags_q[E_BAD_OP] <= 1'b1;
                  end else if (bad_w) begin
                    err_flags_q[E_BAD_WORDS] <= 1'b1;
                  end else begin
                    perf_cycles_q <= 32'd0;
                    done_st_q     <= 1'b0;
                    idx_q         <= '0;
                    bit_cnt_q     <= 13'd0;
                    swap_q        <= 1'b0;
                    uop_pc_q      <= 6'd0;

                    if (is_ecc) begin
                      n_eff_q       <= (ADDR_W+1)'(8);
                      n_eff_m1_q    <= ADDR_W'(7);
                      ecc_is_p256_q <= (ctrl_op_q != OP_X25519_SCALAR);
                      p1_nz_q       <= 1'b0;
                      p2_nz_q       <= 1'b0;
                      if (ctrl_op_q == OP_X25519_SCALAR) seq_q <= ST_X25_INIT;
                      else                               seq_q <= ST_P256_INIT;
                    end else begin
                      n_eff_q       <= num_words_q;
                      n_eff_m1_q    <= ADDR_W'(num_words_q - (ADDR_W+1)'(1));
                      ecc_is_p256_q <= 1'b0;
                      unique case (ctrl_op_q)
                        OP_MONT_MUL: seq_q <= ST_CHK_MOD_PRE;
                        OP_MOD_ADD,
                        OP_MOD_SUB:  seq_q <= ST_SINGLE_RUN;
                        OP_MOD_EXP: begin
                          automatic logic [12:0] full_bits = 13'({num_words_q, 5'd0});
                          bit_idx_q <= ((exp_bits_q == 13'd0) || (exp_bits_q > full_bits)) ?
                                       (full_bits - 13'd1) : (exp_bits_q - 13'd1);
                          seq_q <= ST_CHK_MOD_PRE;
                        end
                        default: seq_q <= ST_IDLE;
                      endcase
                    end
                  end
                end
              end
            end
            default: ;
          endcase
        end else if (engine_busy) begin
          err_flags_q[E_BUSY_WR] <= 1'b1;
        end
      end

      // Sequencer Execution (skipped on WIPE cmd cycle so ST_WIPE starts cleanly)
      if (!(csr_wr_en && sel_csr && (csr_addr == A_CMD) && csr_req_i.wdata[1])) begin
        unique case (seq_q)
          ST_IDLE: ;

          ST_WIPE: begin
            if (idx_q < ADDR_W'(MAX_WORDS - 1)) begin
              idx_q <= idx_q + ADDR_W'(1);
            end
            if (bit_cnt_q < 13'd127) begin
              bit_cnt_q <= bit_cnt_q + 13'd1;
            end else if (idx_q == ADDR_W'(MAX_WORDS - 1)) begin
              wipe_done_st_q <= 1'b1;
              seq_q          <= ST_IDLE;
            end
          end

          ST_CHK_MOD_PRE: seq_q <= ST_CHK_MOD_WAIT;
          ST_CHK_MOD_WAIT: begin
            if (rd_m[0] == 1'b0) begin
              err_flags_q[E_EVEN_MOD] <= 1'b1;
              seq_q                   <= ST_IDLE;
            end else if (ctrl_op_q == OP_MONT_MUL) begin
              seq_q <= ctrl_precomp_en_q ? ST_SINGLE_N0_WAIT : ST_SINGLE_RUN;
            end else begin
              seq_q <= ctrl_precomp_en_q ? ST_PRE_N0_WAIT : ST_PRE_BYPASS_ONE;
            end
          end

          ST_SINGLE_N0_WAIT: begin
            if (alu_done) begin
              n0_inv_hw_q <= alu_n0_inv_out;
              seq_q       <= ST_SINGLE_RUN;
            end
          end

          ST_SINGLE_RUN: begin
            if (alu_done) begin
              done_st_q <= 1'b1;
              seq_q     <= ST_IDLE;
            end
          end

          // ----- ModExp Precompute -----
          ST_PRE_N0_WAIT: begin
            if (alu_done) begin
              n0_inv_hw_q <= alu_n0_inv_out;
              idx_q       <= '0;
              seq_q       <= ST_PRE_INIT_T0;
            end
          end

          ST_PRE_INIT_T0: begin
            if (idx_q == n_eff_m1_q) begin
              idx_q     <= '0;
              bit_cnt_q <= 13'({n_eff_q, 5'd0}) - 13'd1;
              seq_q     <= ST_PRE_DBL_R;
            end else begin
              idx_q <= idx_q + ADDR_W'(1);
            end
          end

          ST_PRE_DBL_R: begin
            if (alu_done) begin
              if (bit_cnt_q == 13'd0) begin
                idx_q     <= '0;
                bit_cnt_q <= 13'd0;
                seq_q     <= ST_PRE_COPY_T0_T1;
              end else begin
                bit_cnt_q <= bit_cnt_q - 13'd1;
              end
            end
          end

          ST_PRE_COPY_T0_T1: begin
            if (bit_cnt_q[0] == 1'b0) begin
              bit_cnt_q[0] <= 1'b1;
            end else begin
              bit_cnt_q[0] <= 1'b0;
              if (idx_q == n_eff_m1_q) begin
                bit_cnt_q <= 13'd31; // 32 doublings for 2^32 * R mod M
                idx_q     <= '0;
                seq_q     <= ST_PRE_DBL_2P32;
              end else begin
                idx_q <= idx_q + ADDR_W'(1);
              end
            end
          end

          ST_PRE_DBL_2P32: begin
            if (alu_done) begin
              if (bit_cnt_q == 13'd0) begin
                idx_q     <= '0;
                bit_cnt_q <= 13'd0;
                seq_q     <= ST_PRE_COPY_T0_T3;
              end else begin
                bit_cnt_q <= bit_cnt_q - 13'd1;
              end
            end
          end

          ST_PRE_COPY_T0_T3: begin
            if (bit_cnt_q[0] == 1'b0) begin
              bit_cnt_q[0] <= 1'b1;
            end else begin
              bit_cnt_q[0] <= 1'b0;
              if (idx_q == n_eff_m1_q) begin
                idx_q       <= '0;
                bit_cnt_q   <= 13'd0;
                n_exp_bit_q <= msb_idx(n_eff_q);
                seq_q       <= ST_PRE_EXP_SQ;
              end else begin
                idx_q <= idx_q + ADDR_W'(1);
              end
            end
          end

          ST_PRE_EXP_SQ: begin
            if (alu_done) begin
              automatic logic [15:0] n_eff_pad = 16'(n_eff_q);
              if (n_eff_pad[n_exp_bit_q]) begin
                seq_q <= ST_PRE_EXP_MUL;
              end else if (n_exp_bit_q == 4'd0) begin
                seq_q <= ST_PRE_CONV_A;
              end else begin
                n_exp_bit_q <= n_exp_bit_q - 4'd1;
              end
            end
          end

          ST_PRE_EXP_MUL: begin
            if (alu_done) begin
              if (n_exp_bit_q == 4'd0) begin
                seq_q <= ST_PRE_CONV_A;
              end else begin
                n_exp_bit_q <= n_exp_bit_q - 4'd1;
                seq_q       <= ST_PRE_EXP_SQ;
              end
            end
          end

          ST_PRE_BYPASS_ONE: begin
            if (alu_done) seq_q <= ST_PRE_CONV_A;
          end

          ST_PRE_CONV_A: begin
            if (alu_done) seq_q <= ST_EXP_RD_E_PRE;
          end

          // ----- ModExp Constant-Time Montgomery Ladder -----
          ST_EXP_RD_E_PRE:  seq_q <= ST_EXP_RD_E_WAIT;
          ST_EXP_RD_E_WAIT: begin
            e_bit_q <= rd_e[bit_idx_q[4:0]];
            seq_q   <= ST_EXP_MUL_CROSS;
          end

          ST_EXP_MUL_CROSS: begin
            if (alu_done) seq_q <= ST_EXP_MUL_SQ;
          end

          ST_EXP_MUL_SQ: begin
            if (alu_done) seq_q <= ST_EXP_SWAP_PRE;
          end

          ST_EXP_SWAP_PRE: begin
            idx_q <= '0;
            seq_q <= ST_EXP_SWAP;
          end

          ST_EXP_SWAP: begin
            if (idx_q == n_eff_m1_q) begin
              if (bit_idx_q == 13'd0) seq_q <= ST_EXP_FINAL;
              else begin
                bit_idx_q <= bit_idx_q - 13'd1;
                seq_q     <= ST_EXP_RD_E_PRE;
              end
            end else begin
              idx_q <= idx_q + ADDR_W'(1);
            end
          end

          ST_EXP_FINAL: begin
            if (alu_done) begin
              done_st_q <= 1'b1;
              seq_q     <= ST_IDLE;
            end
          end

          // ----- RFC 7748 X25519 -----
          ST_X25_INIT: begin
            // bit_cnt_q[3:0] = phase 0..6, idx_q[2:0] = word 0..7
            if (bit_cnt_q[3:0] == 4'd5) begin
              // 1-cycle read wait for A[idx_q]
              bit_cnt_q <= 13'd6;
            end else if (bit_cnt_q[3:0] == 4'd6) begin
              if (idx_q[2:0] == 3'd7) begin
                idx_q     <= '0;
                bit_cnt_q <= 13'd0;
                seq_q     <= ST_X25_RED_U;
              end else begin
                idx_q     <= idx_q + ADDR_W'(1);
                bit_cnt_q <= 13'd5;
              end
            end else begin
              if (idx_q[2:0] == 3'd7) begin
                idx_q     <= '0;
                bit_cnt_q <= bit_cnt_q + 13'd1;
              end else begin
                idx_q <= idx_q + ADDR_W'(1);
              end
            end
          end

          ST_X25_RED_U:  if (alu_done) seq_q <= ST_X25_CONV_U;
          ST_X25_CONV_U: if (alu_done) begin idx_q <= '0; bit_cnt_q <= 13'd0; seq_q <= ST_X25_COPY_X1; end

          ST_X25_COPY_X1: begin
            if (bit_cnt_q[0] == 1'b0) begin
              bit_cnt_q[0] <= 1'b1;
            end else begin
              bit_cnt_q[0] <= 1'b0;
              if (idx_q[2:0] == 3'd7) begin
                idx_q     <= '0;
                bit_idx_q <= 13'd254;
                swap_q    <= 1'b0;
                seq_q     <= ST_X25_RD_E_PRE;
              end else begin
                idx_q <= idx_q + ADDR_W'(1);
              end
            end
          end

          ST_X25_RD_E_PRE: seq_q <= ST_X25_RD_E_WAIT;
          ST_X25_RD_E_WAIT: begin
            automatic logic kt;
            if (bit_idx_q == 13'd254)     kt = 1'b1;
            else if (bit_idx_q <= 13'd2)  kt = 1'b0;
            else                          kt = rd_e[bit_idx_q[4:0]];
            cswap_cond_q <= swap_q ^ kt;
            swap_q       <= kt;
            cswap_cnt_q  <= 3'd0;
            cswap_sa_q   <= S_X0;
            cswap_sb_q   <= S_X1;
            idx_q        <= '0;
            uop_pc_q     <= 6'd0;
            cswap_ret_q  <= ST_X25_STEP_UOP;
            seq_q        <= ST_CSWAP_RD;
          end

          ST_X25_STEP_UOP: begin
            if (cur_uop.op == UOP_DONE) begin
              if (bit_idx_q == 13'd0) begin
                cswap_cond_q <= swap_q;
                cswap_cnt_q  <= 3'd0;
                cswap_sa_q   <= S_X0;
                cswap_sb_q   <= S_X1;
                idx_q        <= '0;
                inv_exp_q    <= P25519_PM2;
                cswap_ret_q  <= ST_INV_PREP;
                seq_q        <= ST_CSWAP_RD;
              end else begin
                bit_idx_q <= bit_idx_q - 13'd1;
                seq_q     <= ST_X25_RD_E_PRE;
              end
            end else if (alu_done) begin
              uop_pc_q <= uop_pc_q + 6'd1;
            end
          end

          // ----- NIST P-256 -----
          ST_P256_INIT: begin
            // Check infinity on A/B (p1_nz_q) and QX/QY (p2_nz_q) during phases 5..7
            if (bit_cnt_q[3:0] == 4'd5) begin
              if (ctrl_op_q == OP_ECC_P256_PMUL && ctrl_ecc_use_g_q) begin
                if (idx_q[2:0] == 3'd7) begin idx_q <= '0; bit_cnt_q <= 13'd6; end
                else idx_q <= idx_q + ADDR_W'(1);
              end else begin
                // Read A[idx], B[idx], QX[idx], QY[idx] across idx 0..7 -> next cycle check nz and write
                bit_cnt_q <= 13'd6;
              end
            end else if (bit_cnt_q[3:0] == 4'd6) begin
              if (ctrl_op_q == OP_ECC_P256_PMUL && ctrl_ecc_use_g_q) begin
                if (idx_q[2:0] == 3'd7) begin idx_q <= '0; bit_cnt_q <= 13'd7; end
                else idx_q <= idx_q + ADDR_W'(1);
              end else begin
                if (rd_a != 32'd0 || rd_b != 32'd0) p1_nz_q <= 1'b1;
                if (rd_ecc_a != 32'd0 || rd_ecc_b != 32'd0) p2_nz_q <= 1'b1;
                bit_cnt_q <= 13'd7;
              end
            end else if (bit_cnt_q[3:0] == 4'd7) begin
              if (ctrl_op_q == OP_ECC_P256_PMUL && ctrl_ecc_use_g_q) begin
                if (idx_q[2:0] == 3'd7) begin
                  // G is already in Montgomery domain! Jump directly to ladder!
                  idx_q     <= '0;
                  bit_cnt_q <= 13'd0;
                  bit_idx_q <= 13'd255;
                  swap_q    <= 1'b0;
                  seq_q     <= ST_P256_RD_E_PRE;
                end else begin
                  idx_q <= idx_q + ADDR_W'(1);
                end
              end else begin
                if (idx_q[2:0] == 3'd7) begin idx_q <= '0; bit_cnt_q <= 13'd8; end
                else begin idx_q <= idx_q + ADDR_W'(1); bit_cnt_q <= 13'd5; end
              end
            end else if (bit_cnt_q[3:0] == 4'd11) begin
              if (idx_q[2:0] == 3'd7) begin
                idx_q     <= '0;
                bit_cnt_q <= 13'd0;
                if (ctrl_op_q == OP_ECC_P256_PMUL)      seq_q <= ST_P256_CONV_P2X;
                else                                    seq_q <= ST_P256_CONV_P1X;
              end else begin
                idx_q <= idx_q + ADDR_W'(1);
              end
            end else begin
              if (idx_q[2:0] == 3'd7) begin idx_q <= '0; bit_cnt_q <= bit_cnt_q + 13'd1; end
              else idx_q <= idx_q + ADDR_W'(1);
            end
          end

          ST_P256_CONV_P1X: if (alu_done) seq_q <= ST_P256_CONV_P1Y;
          ST_P256_CONV_P1Y: if (alu_done) begin
            if (ctrl_op_q == OP_ECC_P256_PADD) seq_q <= ST_P256_CONV_P2X;
            else begin
              p256_dbl_q <= 1'b1;
              uop_pc_q   <= 6'd0;
              seq_q      <= ST_P256_RCB_UOP;
            end
          end
          ST_P256_CONV_P2X: if (alu_done) seq_q <= ST_P256_CONV_P2Y;
          ST_P256_CONV_P2Y: if (alu_done) begin
            if (ctrl_op_q == OP_ECC_P256_PMUL) begin
              bit_idx_q <= 13'd255;
              swap_q    <= 1'b0;
              seq_q     <= ST_P256_RD_E_PRE;
            end else begin
              p256_dbl_q <= 1'b0;
              uop_pc_q   <= 6'd0;
              seq_q      <= ST_P256_RCB_UOP;
            end
          end

          ST_P256_RD_E_PRE: seq_q <= ST_P256_RD_E_WAIT;
          ST_P256_RD_E_WAIT: begin
            automatic logic kt = rd_e[bit_idx_q[4:0]];
            cswap_cond_q <= swap_q ^ kt;
            swap_q       <= kt;
            cswap_cnt_q  <= 3'd2; // 3 pairs: X, Y, Z
            cswap_sa_q   <= S_X0;
            cswap_sb_q   <= S_X1;
            idx_q        <= '0;
            p256_dbl_q   <= 1'b0;
            uop_pc_q     <= 6'd0;
            cswap_ret_q  <= ST_P256_RCB_UOP;
            seq_q        <= ST_CSWAP_RD;
          end

          ST_P256_RCB_UOP: begin
            if (cur_uop.op == UOP_DONE) begin
              if (ctrl_op_q == OP_ECC_P256_PMUL) begin
                idx_q     <= '0;
                bit_cnt_q <= 13'd0;
                seq_q     <= ST_P256_COPY_OUT;
              end else begin
                // PADD / PDBL finished RCB -> affine conversion from T5,T6,T7
                idx_q     <= '0;
                bit_cnt_q <= 13'd0;
                inv_exp_q <= P256_PM2;
                seq_q     <= ST_INV_PREP;
              end
            end else if (alu_done) begin
              uop_pc_q <= uop_pc_q + 6'd1;
            end
          end

          ST_P256_COPY_OUT: begin
            if (bit_cnt_q[0] == 1'b0) begin
              bit_cnt_q[0] <= 1'b1;
            end else begin
              bit_cnt_q[0] <= 1'b0;
              if (idx_q[2:0] == 3'd7) begin
                idx_q <= '0;
                if (bit_cnt_q[2:1] == 2'd2) begin
                  bit_cnt_q <= 13'd0;
                  if (p256_dbl_q == 1'b0) begin
                    // Finished R1 = R0 + R1; now compute R0 = R0 + R0
                    p256_dbl_q <= 1'b1;
                    uop_pc_q   <= 6'd0;
                    seq_q      <= ST_P256_RCB_UOP;
                  end else begin
                    // Finished bit_idx_q
                    if (bit_idx_q == 13'd0) begin
                      cswap_cond_q <= swap_q;
                      cswap_cnt_q  <= 3'd2;
                      cswap_sa_q   <= S_X0;
                      cswap_sb_q   <= S_X1;
                      idx_q        <= '0;
                      cswap_ret_q  <= ST_P256_COPY_R0_T567;
                      seq_q        <= ST_CSWAP_RD;
                    end else begin
                      bit_idx_q <= bit_idx_q - 13'd1;
                      seq_q     <= ST_P256_RD_E_PRE;
                    end
                  end
                end else begin
                  bit_cnt_q[2:1] <= bit_cnt_q[2:1] + 2'd1;
                end
              end else begin
                idx_q <= idx_q + ADDR_W'(1);
              end
            end
          end

          ST_P256_COPY_R0_T567: begin
            if (bit_cnt_q[0] == 1'b0) begin
              bit_cnt_q[0] <= 1'b1;
            end else begin
              bit_cnt_q[0] <= 1'b0;
              if (idx_q[2:0] == 3'd7) begin
                idx_q <= '0;
                if (bit_cnt_q[2:1] == 2'd2) begin
                  bit_cnt_q <= 13'd0;
                  inv_exp_q <= P256_PM2;
                  seq_q     <= ST_INV_PREP;
                end else begin
                  bit_cnt_q[2:1] <= bit_cnt_q[2:1] + 2'd1;
                end
              end else begin
                idx_q <= idx_q + ADDR_W'(1);
              end
            end
          end

          // ----- Shared Constant-Time Slot Swap -----
          ST_CSWAP_RD:  seq_q <= ST_CSWAP_WR1;
          ST_CSWAP_WR1: begin
            cswap_va_q <= rd_ecc_a;
            cswap_vb_q <= rd_ecc_b;
            seq_q      <= ST_CSWAP_WR2;
          end
          ST_CSWAP_WR2: begin
            if (idx_q[2:0] == 3'd7) begin
              idx_q <= '0;
              if (cswap_cnt_q == 3'd2) begin
                // 3-pair (X,Y,Z) -> next is Y0/Y1
                cswap_sa_q  <= S_Y0;
                cswap_sb_q  <= S_Y1;
                cswap_cnt_q <= 3'd1;
                seq_q       <= ST_CSWAP_RD;
              end else if (cswap_cnt_q == 3'd1) begin
                cswap_sa_q  <= S_Z0;
                cswap_sb_q  <= S_Z1;
                cswap_cnt_q <= 3'd0;
                seq_q       <= ST_CSWAP_RD;
              end else if (cswap_sa_q == S_X0 && !ecc_is_p256_q) begin
                // X25519 2-pair: swap Z0/Z1 next
                cswap_sa_q <= S_Z0;
                cswap_sb_q <= S_Z1;
                seq_q      <= ST_CSWAP_RD;
              end else begin
                seq_q <= cswap_ret_q;
              end
            end else begin
              idx_q <= idx_q + ADDR_W'(1);
              seq_q <= ST_CSWAP_RD;
            end
          end

          // ----- Shared Fermat Inversion & Affine Finish -----
          ST_INV_PREP: begin
            if (bit_cnt_q[1:0] == 2'd0) begin
              bit_cnt_q[1:0] <= 2'd1;
            end else if (bit_cnt_q[1:0] == 2'd1) begin
              if (idx_q[2:0] == 3'd7) begin
                idx_q          <= '0;
                bit_cnt_q[1:0] <= 2'd2;
              end else begin
                idx_q          <= idx_q + ADDR_W'(1);
                bit_cnt_q[1:0] <= 2'd0;
              end
            end else begin
              if (idx_q[2:0] == 3'd7) begin
                idx_q     <= '0;
                bit_cnt_q <= 13'd0;
                bit_idx_q <= ecc_is_p256_q ? 13'd255 : 13'd254;
                seq_q     <= ST_INV_SQ;
              end else begin
                idx_q <= idx_q + ADDR_W'(1);
              end
            end
          end

          ST_INV_SQ: begin
            if (alu_done) begin
              if (inv_exp_q[bit_idx_q[7:0]]) begin
                seq_q <= ST_INV_MUL;
              end else if (bit_idx_q == 13'd0) begin
                seq_q <= ecc_is_p256_q ? ST_P256_FIN_X : ST_X25_FIN_MUL;
              end else begin
                bit_idx_q <= bit_idx_q - 13'd1;
              end
            end
          end

          ST_INV_MUL: begin
            if (alu_done) begin
              if (bit_idx_q == 13'd0) begin
                seq_q <= ecc_is_p256_q ? ST_P256_FIN_X : ST_X25_FIN_MUL;
              end else begin
                bit_idx_q <= bit_idx_q - 13'd1;
                seq_q     <= ST_INV_SQ;
              end
            end
          end

          ST_X25_FIN_MUL:    if (alu_done) seq_q <= ST_X25_FIN_UNMONT;
          ST_X25_FIN_UNMONT: if (alu_done) begin done_st_q <= 1'b1; seq_q <= ST_IDLE; end

          ST_P256_FIN_X:        if (alu_done) seq_q <= ST_P256_FIN_Y;
          ST_P256_FIN_Y:        if (alu_done) seq_q <= ST_P256_FIN_UNMONT_X;
          ST_P256_FIN_UNMONT_X: if (alu_done) seq_q <= ST_P256_FIN_UNMONT_Y;
          ST_P256_FIN_UNMONT_Y: if (alu_done) begin done_st_q <= 1'b1; seq_q <= ST_IDLE; end

          default: seq_q <= ST_IDLE;
        endcase
      end
    end
  end

endmodule : pka_core
